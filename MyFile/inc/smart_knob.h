/*
 * SmartKnob firmware-owned haptic control.
 *
 * The public configuration fields intentionally follow scottbez1/smartknob,
 * while the tuning fields describe this motor's current-mode actuator.
 */
#ifndef __SMART_KNOB_H__
#define __SMART_KNOB_H__

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SMART_KNOB_MAX_DETENT_POSITIONS 5U
#define SMART_KNOB_TELEMETRY_PROTOCOL_VERSION 1U
#define SMART_KNOB_TELEMETRY_COMMAND_ID 0x17U
#define SMART_KNOB_TELEMETRY_STATE_TYPE 0x01U
#define SMART_KNOB_TELEMETRY_MOTION_TYPE 0x02U

typedef uint_least16_t pb_size_t;

typedef struct _PB_SmartKnobConfig {
    int32_t position;
    float sub_position_unit;
    uint8_t position_nonce;
    int32_t min_position;
    int32_t max_position;
    float position_width_radians;
    float detent_strength_unit;
    float endstop_strength_unit;
    float snap_point;
    char text[51];
    pb_size_t detent_positions_count;
    int32_t detent_positions[SMART_KNOB_MAX_DETENT_POSITIONS];
    float snap_point_bias;
    int32_t led_hue;
} PB_SmartKnobConfig;

typedef struct {
    /* (P * position_error - D * velocity) * current_scale_a. */
    float p_gain;
    float d_gain;
    float current_scale_a;
    float current_limit_a;
    uint16_t max_current_permille;
    float friction_current_a;
    float click_current_a;
} SmartKnobTuning;

typedef struct {
    PB_SmartKnobConfig config;
    SmartKnobTuning tuning;
} SmartKnobModeConfig;

typedef struct {
    uint8_t active_mode;
    uint8_t flags;
    int32_t current_position;
    float sub_position_unit;
    float shaft_angle_rad;
    float shaft_velocity_rad_s;
    float commanded_current_ma;
    float measured_current_ma;
} SmartKnobRuntimeState;

typedef struct {
    uint8_t sequence;
    uint8_t type;
    uint8_t destination_id;
    uint8_t data[8];
} SmartKnobTelemetryFrame;

/* CAN function indices used by the firmware-owned SmartKnob protocol. */
enum {
    SMART_KNOB_FUNC_MODE = 0x8001,
    SMART_KNOB_FUNC_TELEMETRY_ENABLE = 0x8002,
    SMART_KNOB_FUNC_TELEMETRY_RATE_HZ = 0x8003,
    SMART_KNOB_FUNC_TELEMETRY_HOST_ID = 0x8004,
    SMART_KNOB_FUNC_MODE_COUNT = 0x8005,
    SMART_KNOB_FUNC_PROTOCOL_VERSION = 0x8006,

    SMART_KNOB_FUNC_P_GAIN = 0x8101,
    SMART_KNOB_FUNC_D_GAIN = 0x8102,
    SMART_KNOB_FUNC_CURRENT_SCALE = 0x8103,
    SMART_KNOB_FUNC_CURRENT_LIMIT = 0x8104,
    SMART_KNOB_FUNC_MAX_CURRENT_PERMILLE = 0x8105,
    SMART_KNOB_FUNC_FRICTION_CURRENT = 0x8106,
    SMART_KNOB_FUNC_CLICK_CURRENT = 0x8107,

    SMART_KNOB_FUNC_CUSTOM_POSITION = 0x8201,
    SMART_KNOB_FUNC_CUSTOM_MIN_POSITION = 0x8202,
    SMART_KNOB_FUNC_CUSTOM_MAX_POSITION = 0x8203,
    SMART_KNOB_FUNC_CUSTOM_WIDTH_DEG = 0x8204,
    SMART_KNOB_FUNC_CUSTOM_DETENT_STRENGTH = 0x8205,
    SMART_KNOB_FUNC_CUSTOM_ENDSTOP_STRENGTH = 0x8206,
    SMART_KNOB_FUNC_CUSTOM_SNAP_POINT = 0x8207,
    SMART_KNOB_FUNC_CUSTOM_SNAP_BIAS = 0x8208,
    SMART_KNOB_FUNC_CUSTOM_CLICK_CURRENT = 0x8209,
    SMART_KNOB_FUNC_CUSTOM_FRICTION_CURRENT = 0x820A,
    SMART_KNOB_FUNC_CUSTOM_CURRENT_SCALE = 0x820B,
    SMART_KNOB_FUNC_CUSTOM_P_GAIN = 0x820C,
    SMART_KNOB_FUNC_CUSTOM_D_GAIN = 0x820D,
    SMART_KNOB_FUNC_CUSTOM_LED_HUE = 0x820E,

    SMART_KNOB_FUNC_STATE_POSITION = 0x8301,
    SMART_KNOB_FUNC_STATE_SUB_POSITION = 0x8302,
    SMART_KNOB_FUNC_STATE_COMMAND_CURRENT = 0x8303,
    SMART_KNOB_FUNC_STATE_MEASURED_CURRENT = 0x8304,
};

/* Compatibility entry points used by the existing mode controller. */
void init_smart_knob(void);
void handle_smart_knob(void);
void smart_knob_set_update_rate(float update_rate_hz);

bool smart_knob_select_mode(uint8_t mode_index);
uint8_t smart_knob_active_mode(void);
const SmartKnobModeConfig *smart_knob_active_config(void);

bool smart_knob_read_parameter(uint16_t index, int32_t *value);
bool smart_knob_write_parameter(uint16_t index, int32_t value, uint8_t host_id);

bool smart_knob_telemetry_enabled(void);
bool smart_knob_build_telemetry(uint32_t now_ms, SmartKnobTelemetryFrame *frame);
bool smart_knob_get_runtime_state(SmartKnobRuntimeState *state);

extern int32_t current_position;
extern float latest_sub_position_unit;

#ifdef __cplusplus
}
#endif

#endif
