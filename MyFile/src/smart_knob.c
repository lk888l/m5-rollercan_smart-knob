/*
   smartknob - Copyright 2022 Scott Bezek

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include "smart_knob.h"
#include "mysys.h"
#include "arm_math.h"
#include "motordriver.h"

#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

#define FOC_PID_P 2500
#define FOC_PID_I 0
#define FOC_PID_D 10
#define FOC_PID_OUTPUT_RAMP 100000
#define FOC_PID_LIMIT 10

/*
 * The setting menu receives its position from the same absolute encoder used
 * by FOC. Do not let one bad SPI sample move the virtual detent grid.
 */
#define SMART_KNOB_MAX_INPUT_SPEED_RAD_S 80.0f
#define SMART_KNOB_MIN_INPUT_STEP_RAD     (3.0f * PI / 180.0f)
#define SMART_KNOB_POSITION_FILTER_ALPHA  0.15f
#define SMART_KNOB_SETTLE_TIME_MS          300U
#define SMART_KNOB_D_ERROR_DEADBAND_RAD    (0.15f * PI / 180.0f)
#define SMART_KNOB_MAX_D_ERROR_RAD_S       8.0f

float DEAD_ZONE_DETENT_PERCENT = 0.2;
float DEAD_ZONE_RAD = 1 * PI / 180;

float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

int32_t current_position = 0;
float latest_sub_position_unit = 0;

float idle_check_velocity_ewma = 0;
uint32_t last_idle_start = 0;
uint32_t last_publish = 0;

float current_detent_center = 0;

unsigned long timestamp_prev = 0;
float integral_prev = 0;
float error_prev = 0;
float output_prev;

float torque;

float motor_pid_velocity_p = FOC_PID_P;
float motor_pid_velocity_i = FOC_PID_I;
float motor_pid_velocity_d = FOC_PID_D;
float motor_pid_velocity_output_ramp = FOC_PID_OUTPUT_RAMP;
float motor_pid_velocity_limit = FOC_PID_LIMIT;

float smart_knob_input = 0;
float dead_zone_adjustment = 0;
float angle_to_detent_center = 0;
bool out_of_bounds;
uint32_t idle_delay = 0;
float rad_diff = 0;

static float smart_knob_position_rad;
static float smart_knob_last_raw_position_rad;
static uint32_t smart_knob_last_sample_us;
static uint32_t smart_knob_enable_time_ms;
static bool smart_knob_sample_valid;

PB_SmartKnobConfig config = {
    .position = 0,
    .sub_position_unit = 0,
    .position_nonce = 6,
    .min_position = 0,
    .max_position = -1,
    .position_width_radians = 8.225806452 * PI / 180,
    .detent_strength_unit = 2,
    .endstop_strength_unit = 1,
    .snap_point = 1.1,
    .led_hue = 25,
};

void init_smart_knob(void)
{
    smart_knob_position_rad = mechanical_rad;
    smart_knob_last_raw_position_rad = mechanical_rad;
    smart_knob_last_sample_us = micros();
    smart_knob_enable_time_ms = HAL_GetTick() + SMART_KNOB_SETTLE_TIME_MS;
    smart_knob_sample_valid = true;
    current_detent_center = smart_knob_position_rad;
    timestamp_prev = micros();
    integral_prev = 0;
    error_prev = 0;
    output_prev = 0;
    idle_check_velocity_ewma = 0;
    last_idle_start = 0;
}

static float smart_knob_get_stable_position(void)
{
    uint32_t now_us = micros();
    float sample_time_s = (now_us - smart_knob_last_sample_us) * 1e-6f;
    float raw_position_rad = mechanical_rad;
    float raw_delta_rad = raw_position_rad - smart_knob_last_raw_position_rad;
    float max_delta_rad;

    if (sample_time_s <= 0.0f || sample_time_s > 0.020f) {
        sample_time_s = 0.001f;
    }
    max_delta_rad = fmaxf(SMART_KNOB_MIN_INPUT_STEP_RAD,
                           SMART_KNOB_MAX_INPUT_SPEED_RAD_S * sample_time_s);

    smart_knob_last_sample_us = now_us;
    if (fabsf(raw_delta_rad) <= max_delta_rad) {
        smart_knob_last_raw_position_rad = raw_position_rad;
        smart_knob_position_rad += SMART_KNOB_POSITION_FILTER_ALPHA *
                                   (raw_position_rad - smart_knob_position_rad);
        smart_knob_sample_valid = true;
    } else {
        smart_knob_sample_valid = false;
    }

    return smart_knob_position_rad;
}

float Ts;
unsigned long timestamp_now;

float knob_pid(float error) 
{
    // calculate the time from the last call
    timestamp_now = micros();
    Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional = motor_pid_velocity_p * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    // method uses the antiwindup Foxboro method : https://core.ac.uk/download/pdf/289952713.pdf
    float integral = integral_prev + motor_pid_velocity_i*Ts*0.5f*(error + error_prev);
    // antiwindup - limit the output
    integral = CONSTRAIN(integral, -motor_pid_velocity_limit, motor_pid_velocity_limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float error_delta = error - error_prev;
    if (fabsf(error_delta) < SMART_KNOB_D_ERROR_DEADBAND_RAD) {
        error_delta = 0.0f;
    }
    float derivative = motor_pid_velocity_d * CONSTRAIN(error_delta / Ts,
                                                         -SMART_KNOB_MAX_D_ERROR_RAD_S,
                                                         SMART_KNOB_MAX_D_ERROR_RAD_S);

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    output = CONSTRAIN(output, -motor_pid_velocity_limit, motor_pid_velocity_limit);

    // if output ramp defined
    if(motor_pid_velocity_output_ramp > 0){
        // limit the acceleration by ramping the output
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > motor_pid_velocity_output_ramp)
            output = output_prev + motor_pid_velocity_output_ramp*Ts;
        else if (output_rate < -motor_pid_velocity_output_ramp)
            output = output_prev - motor_pid_velocity_output_ramp*Ts;
    }
    // saving for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}

void handle_smart_knob(void)
{
    float knob_position_rad = smart_knob_get_stable_position();

    if (!smart_knob_sample_valid || (int32_t)(HAL_GetTick() - smart_knob_enable_time_ms) < 0) {
        /* Do not turn an encoder discontinuity or a just-finished calibration
         * into a torque command.  Start cleanly once samples are stable. */
        integral_prev = 0;
        error_prev = 0;
        output_prev = 0;
        timestamp_prev = micros();
        MotorDriverSetCurrentReal(0.0f);
        return;
    }

    // If we are not moving and we're close to the center (but not exactly there), slowly adjust the centerpoint to match the current position
    idle_check_velocity_ewma = motor_rps * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
    if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
        last_idle_start = 0;
    } else {
        if (last_idle_start == 0) {
            last_idle_start = HAL_GetTick();
        }
    }
    idle_delay = HAL_GetTick() - last_idle_start;
    rad_diff = fabsf(knob_position_rad - current_detent_center);
    if (last_idle_start > 0 && idle_delay > IDLE_CORRECTION_DELAY_MILLIS && rad_diff < IDLE_CORRECTION_MAX_ANGLE_RAD) {
        current_detent_center = knob_position_rad * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
    }

    // Check where we are relative to the current nearest detent; update our position if we've moved far enough to snap to another detent
    angle_to_detent_center = knob_position_rad - current_detent_center;
    #if SK_INVERT_ROTATION
        angle_to_detent_center = -knob_position_rad - current_detent_center;
    #endif

    float snap_point_radians = config.position_width_radians * config.snap_point;
    float bias_radians = config.position_width_radians * config.snap_point_bias;
    float snap_point_radians_decrease = snap_point_radians + (current_position <= 0 ? bias_radians : -bias_radians);
    float snap_point_radians_increase = -snap_point_radians + (current_position >= 0 ? -bias_radians : bias_radians); 

    int32_t num_positions = config.max_position - config.min_position + 1;
    if (angle_to_detent_center > snap_point_radians_decrease && (num_positions <= 0 || current_position > config.min_position)) {
        current_detent_center += config.position_width_radians;
        angle_to_detent_center -= config.position_width_radians;
        current_position--;
    } else if (angle_to_detent_center < snap_point_radians_increase && (num_positions <= 0 || current_position < config.max_position)) {
        current_detent_center -= config.position_width_radians;
        angle_to_detent_center += config.position_width_radians;
        current_position++;
    }

    latest_sub_position_unit = -angle_to_detent_center / config.position_width_radians;

    dead_zone_adjustment = CONSTRAIN(
        angle_to_detent_center,
        fmaxf(-config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
        fminf(config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

    out_of_bounds = num_positions > 0 && ((angle_to_detent_center > 0 && current_position == config.min_position) || (angle_to_detent_center < 0 && current_position == config.max_position));
    motor_pid_velocity_limit = 500; //out_of_bounds ? 10 : 3;
    // motor_pid_velocity_p = out_of_bounds ? config.endstop_strength_unit * 4 : config.detent_strength_unit * 4;


    // Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
    if (fabsf(motor_rps) > 8) {
        // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
        MotorDriverSetCurrentReal(0.0f);
    } else {
        smart_knob_input = -angle_to_detent_center + dead_zone_adjustment;
        if (!out_of_bounds && config.detent_positions_count > 0) {
            bool in_detent = false;
            for (uint8_t i = 0; i < config.detent_positions_count; i++) {
                if (config.detent_positions[i] == current_position) {
                    in_detent = true;
                    break;
                }
            }
            if (!in_detent) {
                smart_knob_input = 0;
            }
        }
        torque = knob_pid(smart_knob_input);
        #if SK_INVERT_ROTATION
            torque = -torque;
        #endif
        MotorDriverSetCurrentReal(torque);
    }    
}
